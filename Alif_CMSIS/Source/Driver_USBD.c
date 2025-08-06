/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     Driver_USBD.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     23-April-2025
 * @brief    USB Driver
 * @bug      None.
 * @Note     Keil MDK-Middleware pack required
 ******************************************************************************/

#include "usbd.h"
#include "Driver_USBD.h"

#define EP_NUM(ep_addr)      (ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK)

#define ARM_USBD_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */

typedef struct {
    /* Initialized status: 0 - not initialized, 1 - initialized */
    uint8_t initialized: 1;
    /* Power status:       0 - not powered,     1 - powered   */
    uint8_t powered    : 1;
    /* Reserved (for padding)  */
    uint8_t reserved   : 6;
} DriverStatus_t;

typedef struct {
    ARM_USBD_STATE usbd_state;
    DriverStatus_t drv_status;
} USBD_DRV_info;

/* Driver Version */
static const ARM_DRIVER_VERSION usbd_driver_version = {ARM_USBD_API_VERSION, ARM_USBD_DRV_VERSION};

/* Driver Capabilities */
static const ARM_USBD_CAPABILITIES usbd_driver_capabilities = {
    0, /* vbus_detection */
    0, /* event_vbus_on */
    1, /* event_vbus_off */
    0  /* reserved */
};

USB_DRIVER           usb_drv ATTR_SECTION("usb_dma_buf");
static USBD_DRV_info usbd_drv;

/* Functions  */

/**
 *\fn          ARM_DRIVER_VERSION USBD_GetVersion (void)
 *\brief       Get driver version.
 *\return      ARM_DRIVER_VERSION
 */
static ARM_DRIVER_VERSION ARM_USBD_GetVersion(void)
{
    return usbd_driver_version;
}

/**
 *\fn          ARM_USBD_CAPABILITIES ARM_USBD_GetCapabilities(void)
 *\brief       Get driver capabilities.
 *\return      ARM_USBD_CAPABILITIES
 */
static ARM_USBD_CAPABILITIES ARM_USBD_GetCapabilities(void)
{
    return usbd_driver_capabilities;
}

/**
 *\fn          int32_t ARM_USBD_Initialize(ARM_USBD_SignalDeviceEvent_t   cb_device_event,
               ARM_USBD_SignalEndpointEvent_t cb_endpoint_event)
 *\brief       Initialize USB Device Interface.
 *\param[in]   cb_device_event Pointer to ARM_USBD_SignalDeviceEvent
 *\param[in]   cb_endpoint_event Pointer to ARM_USBD_SignalEndpointEvent
 *\return      \ref execution_status
 */
static int32_t ARM_USBD_Initialize(ARM_USBD_SignalDeviceEvent_t   cb_device_event,
                                   ARM_USBD_SignalEndpointEvent_t cb_endpoint_event)
{
    USB_DRIVER *drv                 = &usb_drv;

    drv->cb_device_event            = cb_device_event;
    drv->cb_endpoint_event          = cb_endpoint_event;
    usbd_drv.drv_status.initialized = 1;
    return ARM_DRIVER_OK;
}

/**
 *\fn          ARM_USBD_Uninitialize
 *\brief       De-initialize USB Device Interface.
 *\param[in]   none
 *\return      execution_status
 */
static int32_t ARM_USBD_Uninitialize(void)
{
    return ARM_DRIVER_OK;
}

/**
 *\fn          int32_t ARM_USBD_PowerControl(ARM_POWER_STATE state)
 *\brief       Control USB Device Interface Power.
 *\param[in]   Power state
 *\return      execution_status
 */
static int32_t ARM_USBD_PowerControl(ARM_POWER_STATE state)
{
    int32_t status;

    switch (state) {
    case ARM_POWER_OFF:
        usbd_drv.drv_status.initialized = 0;
        usbd_drv.drv_status.powered     = 0;
        NVIC_DisableIRQ(USB_IRQ_IRQn);
        NVIC_ClearPendingIRQ(USB_IRQ_IRQn);
        break;
    case ARM_POWER_LOW:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    case ARM_POWER_FULL:
        if (usbd_drv.drv_status.initialized == 0U) {
            return ARM_DRIVER_ERROR;
        }
        status = usbd_initialize(&usb_drv);
        if (status != ARM_DRIVER_OK) {
            return status;
        }
        usbd_drv.drv_status.powered = 1;
        break;
    }

    return ARM_DRIVER_OK;
}

/**
 *\fn          int32_t ARM_USBD_DeviceConnect(void)
 *\brief       Connect USB Device.
 *\param[in]   none
 *\return      execution_status
 */
static int32_t ARM_USBD_DeviceConnect(void)
{
    return usbd_connect(&usb_drv);
}

/**
 *\fn          int32_t ARM_USBD_DeviceDisconnect(void)
 *\brief       Disconnect USB Device.
 *\param[in]   none
 *\return      execution_status
 */

static int32_t ARM_USBD_DeviceDisconnect(void)
{
    if (usbd_drv.drv_status.powered == 0U) {
        return ARM_DRIVER_ERROR;
    }
    usbd_disconnect(&usb_drv);

    return ARM_DRIVER_OK;
}

/**
 *\fn          ARM_USBD_STATE ARM_USBD_DeviceGetState(void)
 *\brief       Get current USB Device State.
 *\param[in]   none
 *\return      ARM_USBD_STATE
 */
static ARM_USBD_STATE ARM_USBD_DeviceGetState(void)
{
    ARM_USBD_STATE state;

    memset(&state, 0, sizeof(ARM_USBD_STATE));
    return state;
}

/**
 *\fn          int32_t ARM_USBD_DeviceRemoteWakeup(void)
 *\brief       Trigger USB Remote Wakeup.
 *\param[in]   none
 *\return      execution_status
 */
static int32_t ARM_USBD_DeviceRemoteWakeup(void)
{
    /* Remote wakeup was disabled  */
    return ARM_DRIVER_OK;
}

/**
 *\fn          int32_t ARM_USBD_DeviceSetAddress(uint8_t dev_addr)
 *\brief       Set USB Device Address.
 *\param[in]   dev_addr  Device Address
 *\return      execution_status
 */
static int32_t ARM_USBD_DeviceSetAddress(uint8_t dev_addr)
{
    return usbd_set_device_address(&usb_drv, dev_addr);
}

/**
 *\fn          int32_t ARM_USBD_ReadSetupPacket(uint8_t *setup)
 *\brief       Read setup packet received over Control Endpoint.
 *\param[out]  setup  Pointer to buffer for setup packet
 *\return      execution_status
 */

static int32_t ARM_USBD_ReadSetupPacket(uint8_t *setup)
{
    memcpy(setup, &usb_drv.setup_data, USB_SETUP_PKT_SIZE);

    return ARM_DRIVER_OK;
}

/**
 *\fn          int32_t ARM_USBD_EndpointConfigure (uint8_t ep_addr, uint8_t ep_type,
               uint16_t ep_max_packet_size)
 *\brief       Configure USB Endpoint.
 *\param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
 *\param[in]   ep_type  Endpoint Type
 *\param[in]   ep_max_packet_size Endpoint Maximum Packet Size
 *\return      execution_status
 */
static int32_t ARM_USBD_EndpointConfigure(uint8_t ep_addr, uint8_t ep_type,
                                          uint16_t ep_max_packet_size)
{
    uint8_t  ep_num;
    uint8_t  ep_dir;

    uint8_t ep_interval = 0;
    /* upper layer has to send ep interval to the low level driver */
    ep_num              = EP_NUM(ep_addr);
    ep_dir              = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) ? USB_DIR_IN : USB_DIR_OUT;

    return usbd_endpoint_create(&usb_drv, ep_type, ep_num, ep_dir, ep_max_packet_size, ep_interval);
}

/**
 *\fn          int32_t ARM_USBD_EndpointUnconfigure(uint8_t ep_addr)
 *\brief       Unconfigure USB Endpoint.
 *\param[in]   ep_addr  Endpoint Address
               - ep_addr.0..3: Address
               - ep_addr.7:    Direction
 *\return      execution_status
 */
static int32_t ARM_USBD_EndpointUnconfigure(uint8_t ep_addr)
{
    uint8_t ep_num;
    uint8_t ep_dir;

    ep_num = EP_NUM(ep_addr);
    ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) ? USB_DIR_IN : USB_DIR_OUT;
    usbd_stop_transfer(&usb_drv, ep_num, ep_dir, true);

    return usbd_ep_disable(&usb_drv, ep_num, ep_dir);
}

/**
 *\fn          int32_t ARM_USBD_EndpointStall(uint8_t ep_addr, bool stall)
 *\brief       Set/Clear Stall for USB Endpoint.
 *\param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
 *\param[in]   stall  Operation
               false Clear the stall
               true Set the stall
 *\return      execution_status
 */
static int32_t ARM_USBD_EndpointStall(uint8_t ep_addr, bool stall)
{
    uint8_t ep_num;
    uint8_t ep_dir;

    ep_num = EP_NUM(ep_addr);
    ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) ? USB_DIR_IN : USB_DIR_OUT;

    return usbd_ep_stall(&usb_drv, ep_num, ep_dir, stall);
}

/**
 *\fn          int32_t ARM_USBD_EndpointTransfer(uint8_t ep_addr, uint8_t *data, uint32_t num)
 *\brief       Read data from or Write data to USB Endpoint.
 *\param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
 *\param[in]   data Pointer to buffer for data to read or data to write
 *\param[in]   num  Number of data bytes to transfer
 *\return      execution_status
 */

static int32_t ARM_USBD_EndpointTransfer(uint8_t ep_addr, uint8_t *data, uint32_t num)
{
    uint8_t  ep_num;
    uint8_t  ep_dir;

    ep_num = EP_NUM(ep_addr);
    ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) ? USB_DIR_IN : USB_DIR_OUT;

    /* For two-stage control transfers:
     * According to the controller data sheet, once the setup packet is received,
     * wait for the XferNotReady event for the status stage.
     * After receiving the XferNotReady (Status) event, proceed to start the status stage.
     */

    /* In Keil MDK usbd lib, after processing the setup packet,
     * immediately start sending the status stage.
     */
    if ((ep_num == USB_CONTROL_EP) && (data == NULL) && (num == 0)) {
        if (usb_drv.cb_endpoint_event != NULL) {
            usb_drv.cb_endpoint_event(ep_num | ARM_USB_ENDPOINT_DIRECTION_MASK, ARM_USBD_EVENT_IN);
        }
        return ARM_DRIVER_OK;
    }
    if (ep_num == 0) {
        if (ep_dir != 0) {
            return usbd_ep0_send(&usb_drv, ep_num, ep_dir, data, num);
        } else {
            return usbd_ep0_recv(&usb_drv, ep_num, ep_dir, data, num);
        }
    } else {
        if (ep_dir != 0) {
            return usbd_bulk_send(&usb_drv, ep_num, ep_dir, data, num);
        } else {
            return usbd_bulk_recv(&usb_drv, ep_num, ep_dir, data, num);
        }
    }
}

/**
 *\fn          uint32_t ARM_USBD_EndpointTransferGetResult(uint8_t ep_addr)
 *\brief       Get result of USB Endpoint transfer.
 *\param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
 *\return      number of successfully transferred data bytes
 */
static uint32_t ARM_USBD_EndpointTransferGetResult(uint8_t ep_addr)
{
    uint8_t  ep_num;
    uint32_t transferred;

    ep_num = EP_NUM(ep_addr);
    if (ep_num == 0) {
        transferred = usb_drv.actual_length;
    } else {
        transferred = usb_drv.num_bytes;
    }

    return transferred;
}

/**
 *\fn          int32_t ARM_USBD_EndpointTransferAbort(uint8_t ep_addr)
 *\brief       Abort current USB Endpoint transfer.
 *\param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
 *\return      execution_status
 */
static int32_t ARM_USBD_EndpointTransferAbort(uint8_t ep_addr)
{
    uint8_t  ep_num;
    uint8_t  ep_dir;

    ep_num = EP_NUM(ep_addr);
    ep_dir = (ep_addr & ARM_USB_ENDPOINT_DIRECTION_MASK) ? USB_DIR_IN : USB_DIR_OUT;

    return usbd_ep_transfer_abort(&usb_drv, ep_num, ep_dir);
}

/**
 *\fn          uint16_t ARM_USBD_GetFrameNumber(void)
 *\brief       Get current USB Frame Number.
 *\param[in]   none
 *\return      Frame Number
 */
static uint16_t ARM_USBD_GetFrameNumber(void)
{
    return ARM_DRIVER_OK;
}

/**
 *\fn          void ARM_USBD_SignalDeviceEvent (uint32_t event)
 *\brief       Signal USB Device Event.
 *\param[in]   event USBD_dev_events
 *\return      none
 */
static void ARM_USBD_SignalDeviceEvent(uint32_t event)
{
    /* function body   */
}

/**
 *\fn          void ARM_USBD_SignalEndpointEvent (uint8_t ep_addr, uint32_t event)
 *\brief       Signal USB Endpoint Event.
 *\param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
 *\param[in]   event USBD_ep_events
 *\return      none
 */

static void ARM_USBD_SignalEndpointEvent(uint8_t ep_addr, uint32_t ep_event)
{
    /* function body  */
}

/* End USBD Interface   */

extern ARM_DRIVER_USBD Driver_USBD0;
ARM_DRIVER_USBD        Driver_USBD0 = {
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
